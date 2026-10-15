// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @cond OVPHYSX_PLC_TRACE */
/**
 * @implements REQ-CAPI-SLEEP-001
 * @covers AC-1
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-4 AC-5
 */
/** @endcond */

#ifndef OVPHYSX_ATTRIBUTES_H
#define OVPHYSX_ATTRIBUTES_H

    /**
     * Canonical physics-output attribute names (semantic, not USD attribute
     * names). Pass any of these to ovphysx_read(). Which names a type
     * produces is documented in the ovstage usage guide.
     *
     * Include this header alongside ovphysx.h. ovphysx.h does not include it.
     */
    #define OVPHYSX_ATTR_POSITION          "position"          /* vec3 f32 (world) */
    #define OVPHYSX_ATTR_ORIENTATION       "orientation"       /* quat f32 xyzw (world) */
    #define OVPHYSX_ATTR_LINEAR_VELOCITY   "linearVelocity"    /* vec3 f32 */
    #define OVPHYSX_ATTR_ANGULAR_VELOCITY  "angularVelocity"   /* vec3 f32 */
    #define OVPHYSX_ATTR_LINEAR_ACCELERATION  "linearAcceleration"  /* vec3 f32 */
    #define OVPHYSX_ATTR_ANGULAR_ACCELERATION "angularAcceleration" /* vec3 f32 */
    /* Body properties. Simulation INPUTS, so these columns are HOST-resident (kDLCPU) even on a
       DirectGPU scene (there is no device copy of an authored value to read) and they have no
       point-instancer array form. centerOfMass* is in the body's LOCAL frame, unlike
       position/orientation which are world. */
    #define OVPHYSX_ATTR_MASS              "mass"              /* f32 */
    #define OVPHYSX_ATTR_INVERSE_MASS      "inverseMass"       /* f32 */
    #define OVPHYSX_ATTR_INERTIA           "inertia"           /* mat3 f32 (9, COM frame) */
    #define OVPHYSX_ATTR_INVERSE_INERTIA   "inverseInertia"    /* mat3 f32 (9, COM frame) */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_POSITION    "centerOfMassPosition"    /* vec3 f32 (local) */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_ORIENTATION "centerOfMassOrientation" /* quat f32 xyzw (local) */
    /* Whole-articulation ROOT-LINK state, for OVPHYSX_QUERY_TARGET_ARTICULATION. Named apart from
       OVPHYSX_ATTR_POSITION and friends because the row is keyed by the prim carrying
       PhysicsArticulationRootAPI, which may be an ancestor Xform rather than the root link itself.
       Unlike every other type, the value is therefore not this row's own prim's pose. */
    #define OVPHYSX_ATTR_ROOT_POSITION         "rootPosition"        /* vec3 f32 (scene world) */
    #define OVPHYSX_ATTR_ROOT_ORIENTATION      "rootOrientation"     /* quat f32 xyzw (scene world) */
    #define OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY  "rootLinearVelocity"  /* vec3 f32 */
    #define OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY "rootAngularVelocity" /* vec3 f32 */
    /* Whole-articulation centres of mass. WORLD uses the same scene-world frame as
       OVPHYSX_ATTR_ROOT_POSITION and is independent of TensorBindings subspace roots. LOCAL uses
       the root link's centre-of-mass (mass) frame, not its actor/prim frame. */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_WORLD "centerOfMassWorld" /* vec3 f32 (scene world) */
    #define OVPHYSX_ATTR_CENTER_OF_MASS_LOCAL "centerOfMassLocal" /* vec3 f32 (root-link mass frame) */
    /* Whole-articulation inverse dynamics, row-major flattened into dtype.lanes. The width depends on the
       articulation's topology, not on the attribute, so a read emits one group per cohort of
       structurally identical articulations and lanes is read per group. JACOBIAN carries its two
       dimensions in the companion JACOBIAN_SHAPE column; the others recover theirs from lanes.
       Generalized joint coordinates follow the authored USD body relationship: their sign is +1
       when body0 is the articulation parent and -1 when body1 is the parent. With S_dof containing
       those signs and T=S_dof for a fixed base or T=diag(I6,S_dof) for a floating base, the returned
       quantities are J=J_physx*T, M=T*M_physx*T, c=T*c_physx, g=T*g_physx, and [A|b]=
       [A_physx*T|b_physx] for the packed centroidal matrix and bias. Angular generalized-coordinate
       dimensions use radians and receive no degree conversion; prismatic and floating-translation
       dimensions retain their linear units. */
    #define OVPHYSX_ATTR_JACOBIAN            "jacobian"           /* f32 [rows*cols] */
    #define OVPHYSX_ATTR_JACOBIAN_SHAPE      "jacobianShape"      /* int32, 2 lanes: rows, cols */
    #define OVPHYSX_ATTR_MASS_MATRIX         "massMatrix"         /* f32 [M*M] */
    #define OVPHYSX_ATTR_CORIOLIS_FORCE      "coriolisForce"      /* f32 [M] */
    #define OVPHYSX_ATTR_GRAVITY_FORCE       "gravityForce"       /* f32 [M] */
    #define OVPHYSX_ATTR_CENTROIDAL_MOMENTUM "centroidalMomentum" /* f32, floating base only */
    #define OVPHYSX_ATTR_DISABLE_GRAVITY    "disableGravity"    /* uint8, nonzero = disabled */
    #define OVPHYSX_ATTR_DISABLE_SIMULATION "disableSimulation" /* uint8, nonzero = disabled */
    /* Borrowed PhysX SDK address, host uint64. The value is a snapshot of the address.
       The object it names is not owned by the caller. See the developer guide. */
    #define OVPHYSX_ATTR_PHYSX_POINTER     "physxPointer"      /* uint64, borrowed address [read-only] */
    /* Kinematic target pose, split like the live pose; only a kinematic body is driven. */
    #define OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION    "kinematicTargetPosition"    /* vec3 f32 (world) */
    #define OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION "kinematicTargetOrientation" /* quat f32 xyzw (world) */
    /* Per-SHAPE properties: one value per shape, padded to the widest selected object in the read, so
       `dtype.lanes` is that width and a row is shapeCount real values then zeros. Read
       OVPHYSX_ATTR_SHAPE_COUNT to find the end. The padding is zero-filled, but 0.0 is a legal
       offset, so it is not a terminator. Host-resident like the properties above. Only the FIRST
       material of a shape is reported. */
    #define OVPHYSX_ATTR_STATIC_FRICTION   "staticFriction"    /* f32 [per shape] */
    #define OVPHYSX_ATTR_DYNAMIC_FRICTION  "dynamicFriction"   /* f32 [per shape] */
    #define OVPHYSX_ATTR_RESTITUTION       "restitution"       /* f32 [per shape] */
    #define OVPHYSX_ATTR_CONTACT_OFFSET    "contactOffset"     /* f32 [per shape] */
    #define OVPHYSX_ATTR_REST_OFFSET       "restOffset"        /* f32 [per shape] */
    #define OVPHYSX_ATTR_SHAPE_COUNT       "shapeCount"        /* int32, one per body or whole articulation */
    /* Whole-articulation topology metadata (read-only): one int32 per articulation, replacing
       ovphysx_articulation_metadata_t field-for-field. linkCount is the count of articulation LINKS
       (the old struct's mislabelled body_count), not standalone rigid bodies. */
    #define OVPHYSX_ATTR_DOF_COUNT            "dofCount"            /* int32: DOFs */
    #define OVPHYSX_ATTR_LINK_COUNT           "linkCount"           /* int32: articulation links */
    #define OVPHYSX_ATTR_JOINT_COUNT          "jointCount"          /* int32: joints */
    #define OVPHYSX_ATTR_FIXED_TENDON_COUNT   "fixedTendonCount"    /* int32: fixed tendons */
    #define OVPHYSX_ATTR_SPATIAL_TENDON_COUNT "spatialTendonCount"  /* int32: spatial tendons */
    #define OVPHYSX_ATTR_IS_FIXED_BASE        "isFixedBase"         /* int32: 1 = fixed base, 0 = floating */
    /* Whole-articulation DOF-column descriptor (read-only): a per-DOF array per articulation, padded to
       the widest selected articulation's dofCount, in PhysX flattened cache-DOF order. Column
       (isFixedBase ? 0 : 6) + k of jacobian / massMatrix maps to DOF k here, so a consumer recovers
       column -> (joint prim, axis). dofJointPath is the owning joint's interned prim-path handle (resolve
       through the attached Stage's path dictionary as for prims.list; 0 pads), replacing the deprecated
       DOF/body/joint name strings; dofAxisIndex is the DOF's ordinal among the joint's UNLOCKED axes
       (0, or 0/1/2 for a fully free spherical joint; a locked axis compacts it), not a canonical axis id. */
    #define OVPHYSX_ATTR_DOF_JOINT_PATH       "dofJointPath"        /* uint64 [array, per DOF]: joint prim handle */
    #define OVPHYSX_ATTR_DOF_AXIS_INDEX       "dofAxisIndex"        /* int32 [array, per DOF]: axis within joint */
    #define OVPHYSX_ATTR_POINTS            "points"            /* vec3 f32 [array] */
    #define OVPHYSX_ATTR_VELOCITIES        "velocities"        /* vec3 f32 [array] */
    #define OVPHYSX_ATTR_REST_POINTS       "restPoints"        /* vec3 f32 [array] (sim-mesh-local) */
    #define OVPHYSX_ATTR_SIM_ELEMENT_INDICES "simElementIndices" /* int32 [array] (4 per tet / 3 per tri) */
    #define OVPHYSX_ATTR_COLLISION_ELEMENT_INDICES "collisionElementIndices" /* int32 [array] (4 per tet) */
    /* Volume-deformable sim-node kinematic target: vec4 f32 [array], one PxVec4 per sim-mesh node
       (xyz target + w active-flag; w==0 drives the node, w!=0 leaves it free). Volume only. */
    #define OVPHYSX_ATTR_KINEMATIC_TARGET  "kinematicTarget"
    /* Deformable material properties: one row per material prim, host-resident. The `deformable`
       prefix disambiguates from the per-SHAPE rigid dynamicFriction above, which is a different
       quantity with a different shape. */
    #define OVPHYSX_ATTR_DEFORMABLE_DYNAMIC_FRICTION   "deformableDynamicFriction"   /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS     "deformableYoungsModulus"     /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_POISSONS_RATIO     "deformablePoissonsRatio"     /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING "deformableElasticityDamping" /* f32 */
    #define OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS  "deformableBendingStiffness"  /* f32, surface only */
    #define OVPHYSX_ATTR_DEFORMABLE_THICKNESS          "deformableThickness"         /* f32, surface only */
    #define OVPHYSX_ATTR_DEFORMABLE_BENDING_DAMPING    "deformableBendingDamping"    /* f32, surface only */

    /* Contact-sensor aggregates: one row per contact sensor (reporting body or, on a CPU scene, reporting
       collider), one component each. The net NORMAL force (contact-point impulses summed) and the net
       FRICTION force (friction-anchor impulses summed)
       PhysX reports on the sensor this step, each divided by the step dt; their sum is the total contact
       force. Friction anchor forces that oppose each other cancel in the net vector, so a friction torque
       does not show in it. Residency follows the scene, like every read column: host (kDLCPU) on a
       non-DirectGPU scene, device (kDLCUDA) on a DirectGPU scene -- branch on the tensor's
       device.device_type before touching the data. */
    #define OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE   "netNormalContactForce"   /* vec3 f32 (per contact sensor) */
    #define OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE "netFrictionContactForce" /* vec3 f32 (per contact sensor) */

    /* Contact-sensor force matrices, read on a contact query with filters (ovphysx_query_contacts): one row
       per sensor, its F filters' forces folded into the lanes, filter f at lanes 3f..3f+2 (F x 3 floats).
       Each cell is the force the sensor receives from that filter's body, normal or friction. Not read on a
       query without filters. */
    #define OVPHYSX_ATTR_NORMAL_CONTACT_FORCE_MATRIX   "normalContactForceMatrix"   /* F x vec3 f32 (per sensor) */
    #define OVPHYSX_ATTR_FRICTION_CONTACT_FORCE_MATRIX "frictionContactForceMatrix" /* F x vec3 f32 (per sensor) */

    /* Contact-sensor filtered detail, read on a contact query with filters: the contact points (normal
       family) and friction anchors (friction family) of each pair between a sensor and one of its filters'
       bodies. Each element attribute is an array group, one tensor per sensor holding its elements ordered
       by filter column (CPU scenes: report order within a column; DirectGPU scenes: unspecified order within
       a column), sized to this step's elements. A sensor with no element this step is absent from these
       groups. The count companions cover every sensor, F int32 lanes: lane f is the number of the sensor's
       elements from filter f. Forces are divided by the step dt and negated where the sensor is the pair's
       second body; normals are as reported. */
    #define OVPHYSX_ATTR_FILTERED_CONTACT_FORCE      "filteredContactForce"      /* f32 (array per sensor) */
    #define OVPHYSX_ATTR_FILTERED_CONTACT_POINT      "filteredContactPoint"      /* vec3 f32 (array per sensor) */
    #define OVPHYSX_ATTR_FILTERED_CONTACT_NORMAL     "filteredContactNormal"     /* vec3 f32 (array per sensor) */
    #define OVPHYSX_ATTR_FILTERED_CONTACT_SEPARATION "filteredContactSeparation" /* f32 (array per sensor) */
    #define OVPHYSX_ATTR_FILTERED_CONTACT_COUNT      "filteredContactCount"      /* F x i32 (per sensor) */
    #define OVPHYSX_ATTR_FILTERED_FRICTION_FORCE     "filteredFrictionForce"     /* vec3 f32 (array per sensor) */
    #define OVPHYSX_ATTR_FILTERED_FRICTION_POINT     "filteredFrictionPoint"     /* vec3 f32 (array per sensor) */
    #define OVPHYSX_ATTR_FILTERED_FRICTION_COUNT     "filteredFrictionCount"     /* F x i32 (per sensor) */

    /* Contact-sensor raw contacts, read on every contact-sensor query: every contact point of every pair a
       sensor is in, whatever the other body. Each is an array group, one tensor per sensor holding its points
       (CPU scenes: report order; DirectGPU scenes: unspecified order), sized to this step's points; a sensor
       with no point this step is absent. Forces are divided by the step dt and negated where the sensor is
       the pair's second body; normals are as reported. The other body is its prim's interned path handle
       (ovx_primpath_t, resolved through the stage's path dictionary; 0 when the body has none). */
    #define OVPHYSX_ATTR_RAW_CONTACT_FORCE      "rawContactForce"      /* f32 (array per sensor) */
    #define OVPHYSX_ATTR_RAW_CONTACT_POINT      "rawContactPoint"      /* vec3 f32 (array per sensor) */
    #define OVPHYSX_ATTR_RAW_CONTACT_NORMAL     "rawContactNormal"     /* vec3 f32 (array per sensor) */
    #define OVPHYSX_ATTR_RAW_CONTACT_SEPARATION "rawContactSeparation" /* f32 (array per sensor) */
    #define OVPHYSX_ATTR_RAW_CONTACT_OTHER_BODY "rawContactOtherBody"  /* u64 prim handle (array per sensor) */

    /* Contact-sensor events, read on every contact-sensor query over a CPU scene (a DirectGPU scene returns no
       event group: PhysX's GPU contact pairs carry no event type): each contact-report pair a sensor is a side
       of -- found, persisting or lost -- as seen from that sensor, in report order. Each is an array group at
       one of three levels: one element per event (the names without POINT/ANCHOR, and the two counts
       CONTACT_EVENT_POINT_COUNT and CONTACT_EVENT_ANCHOR_COUNT), one per contact point (the other POINT
       names) or one per friction anchor (the other ANCHOR names), points and anchors concatenated in event
       order and split by those two counts. A sensor has a tensor when it has at least one element at the
       name's level this step, so a sensor whose only event is a lost pair, which carries no point, has event
       tensors but no point or anchor tensor. A pair between two sensors appears under each. Identities are
       interned prim-path handles (0 when none), ordered own then other; values are as reported (impulses,
       never negated). The type is 0 found, 1 lost, 2 persist. The
       PROTO_INDEX names hold the collider's instance index in its point instancer -- its slot in
       protoIndices, not the prototype that slot selects -- and 0xFFFFFFFF when the collider is not an
       instance. */
    #define OVPHYSX_ATTR_CONTACT_EVENT_TYPE                   "contactEventType"                /* i32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_SENSOR_SIDE            "contactEventSensorSide"          /* 0 or 1 (i32) */
    #define OVPHYSX_ATTR_CONTACT_EVENT_COLLIDER               "contactEventCollider"            /* u64 prim handle */
    #define OVPHYSX_ATTR_CONTACT_EVENT_OTHER_BODY             "contactEventOtherBody"           /* u64 prim handle */
    #define OVPHYSX_ATTR_CONTACT_EVENT_OTHER_COLLIDER         "contactEventOtherCollider"       /* u64 prim handle */
    #define OVPHYSX_ATTR_CONTACT_EVENT_PROTO_INDEX            "contactEventProtoIndex"          /* u32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_OTHER_PROTO_INDEX      "contactEventOtherProtoIndex"     /* u32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_COUNT            "contactEventPointCount"          /* i32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_COUNT           "contactEventAnchorCount"         /* i32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_POSITION         "contactEventPointPosition"       /* vec3 f32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_NORMAL           "contactEventPointNormal"         /* vec3 f32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_IMPULSE          "contactEventPointImpulse"        /* vec3 f32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_SEPARATION       "contactEventPointSeparation"     /* f32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_FACE_INDEX       "contactEventPointFaceIndex"      /* u32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_OTHER_FACE_INDEX "contactEventPointOtherFaceIndex" /* u32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_MATERIAL         "contactEventPointMaterial"       /* u64 prim handle */
    #define OVPHYSX_ATTR_CONTACT_EVENT_POINT_OTHER_MATERIAL   "contactEventPointOtherMaterial"  /* u64 prim handle */
    #define OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_POSITION        "contactEventAnchorPosition"      /* vec3 f32 */
    #define OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_IMPULSE         "contactEventAnchorImpulse"       /* vec3 f32 */

    /* Articulation DOF attributes: one value per ENABLED AXIS of the joint prim. Angular axes are
       reported in DEGREES (what USD authored) and signed quantities carry the joint's body-order
       sign. The `joint` prefix disambiguates from the per-SHAPE rigid staticFriction /
       dynamicFriction above, which are different quantities. */
    #define OVPHYSX_ATTR_JOINT_POSITION    "jointPosition"     /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_VELOCITY    "jointVelocity"     /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_POSITION_TARGET "jointPositionTarget" /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_VELOCITY_TARGET "jointVelocityTarget" /* f32 [array, per-axis] */
    #define OVPHYSX_ATTR_JOINT_ACTUATION_FORCE "jointActuationForce" /* f32 [array, per-axis] */
    /* The link's incoming joint force resolved onto this DOF's own axis. Read-only. */
    #define OVPHYSX_ATTR_JOINT_PROJECTED_FORCE "jointProjectedForce" /* f32 [array, per-axis], read-only */
    /* DOF properties: simulation INPUTS, so HOST-resident (kDLCPU) columns even on a DirectGPU
       scene. One read can mix a kDLCUDA jointPosition with a kDLCPU jointStiffness. */
    #define OVPHYSX_ATTR_JOINT_STIFFNESS   "jointStiffness"    /* f32: drive effort per unit of position error */
    #define OVPHYSX_ATTR_JOINT_DAMPING     "jointDamping"      /* f32: drive effort per unit of rate error */
    #define OVPHYSX_ATTR_JOINT_LIMIT       "jointLimit"        /* f32 x2 (low, high): one interval, FLT_MAX when free */
    #define OVPHYSX_ATTR_JOINT_MAX_VELOCITY "jointMaxVelocity" /* f32: the JOINT's rate bound */
    #define OVPHYSX_ATTR_JOINT_MAX_FORCE   "jointMaxForce"     /* f32: the drive's effort bound */
    #define OVPHYSX_ATTR_JOINT_ARMATURE    "jointArmature"     /* f32: added rotor inertia on the axis */
    #define OVPHYSX_ATTR_JOINT_STATIC_FRICTION  "jointStaticFriction"  /* f32: break-away effort */
    #define OVPHYSX_ATTR_JOINT_DYNAMIC_FRICTION "jointDynamicFriction" /* f32: sliding effort */
    #define OVPHYSX_ATTR_JOINT_VISCOUS_FRICTION "jointViscousFriction" /* f32: effort per unit rate, a
                                                                          coefficient, unlike the two efforts above */
    #define OVPHYSX_ATTR_JOINT_SPEED_EFFORT_GRADIENT "jointSpeedEffortGradient" /* f32: drive envelope, rate per unit effort */
    #define OVPHYSX_ATTR_JOINT_MAX_ACTUATOR_VELOCITY "jointMaxActuatorVelocity" /* f32: the ACTUATOR's rate bound,
                                                                                   not the joint's above */
    #define OVPHYSX_ATTR_JOINT_VELOCITY_DEPENDENT_RESISTANCE                                                           \
        "jointVelocityDependentResistance" /* f32: drive envelope, resisting effort per unit rate */
    #define OVPHYSX_ATTR_JOINT_DRIVE_TYPE  "jointDriveType"    /* uint8 [array, per-axis]: 0 none, 1 force, 2 accel */
    /* On OVPHYSX_QUERY_TARGET_ARTICULATION_LINK: the spatial force the inbound joint exerts on the link,
       spelled as in PxArticulationCache::linkIncomingJointForce. */
    #define OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE "linkIncomingJointForce" /* f32 x6: force xyz, torque xyz, joint frame */
    /* Tendon properties: authoring-time values, so they change only when the tendon does. */
    #define OVPHYSX_ATTR_TENDON_STIFFNESS       "tendonStiffness"       /* f32 */
    #define OVPHYSX_ATTR_TENDON_DAMPING         "tendonDamping"         /* f32 */
    #define OVPHYSX_ATTR_TENDON_LIMIT_STIFFNESS "tendonLimitStiffness"  /* f32 */
    #define OVPHYSX_ATTR_TENDON_LIMIT           "tendonLimit"           /* f32 x2 (low, high), fixed tendons only */
    #define OVPHYSX_ATTR_TENDON_REST_LENGTH     "tendonRestLength"      /* f32, fixed tendons only */
    #define OVPHYSX_ATTR_TENDON_OFFSET          "tendonOffset"          /* f32 */

    /* Sleep state is separate from the wake command: one host uint8 per rigid body or articulation
       link (the owning articulation's state), 1 asleep / 0 awake. Kinematic and disabled bodies on
       CPU/readback scenes report their PhysX state. Scenes with sleeping disabled report 0; DirectGPU retains its
       step-first and disabled-row omission rules. No point-instancer array form. Read-only. */
    #define OVPHYSX_ATTR_IS_SLEEPING            "isSleeping"

    /* Write-only control inputs (ADR-0012). Accepted by ovphysx_write, never emitted by
       ovphysx_read: the solver consumes and clears them each step, so there is no stored
       value to report back. */
    #define OVPHYSX_ATTR_FORCE                  "force"                 /* vec3 f32, at the COM [write-only] */
    #define OVPHYSX_ATTR_WRENCH                 "wrench"                /* 9 f32: force, torque, world point [write-only] */
    /* wake is a one-time, write-only command: nonzero wakes the addressed rigid dynamics, zero
       sleeps them. Later simulation can change their state. CONDITIONAL in ovphysx_writability:
       supported on CPU scenes and GPU scenes with CPU readback; refused on DirectGPU scenes. */
    #define OVPHYSX_ATTR_WAKE                   "wake"                  /* uint8, nonzero = wake / zero = sleep [write-only] */

    /* Whole-vehicle data: fixed CPU columns, one row per vehicle prim. The chassis pose and
       velocity remain on OVPHYSX_QUERY_TARGET_RIGID_BODY. Managed controls require DriveBasic or
       DriveStandard; transmission telemetry requires DriveStandard. Ordinary controls persist
       until rewritten, including across resetToRestState. requestedGear is a command and is not
       the transmission's internal targetGear or currentGear. */
    #define OVPHYSX_ATTR_ACCELERATOR              "accelerator"              /* f32, [0,1], read/write */
    #define OVPHYSX_ATTR_BRAKE0                   "brake0"                   /* f32, [0,1], read/write */
    #define OVPHYSX_ATTR_BRAKE1                   "brake1"                   /* f32, [0,1], read/write */
    #define OVPHYSX_ATTR_STEER                    "steer"                    /* f32, [-1,1], read/write */
    #define OVPHYSX_ATTR_REQUESTED_GEAR           "requestedGear"            /* int32, [-1,30] or 255, write-only */
    #define OVPHYSX_ATTR_THRUST0                  "thrust0"                  /* f32, [-1,1], tank-only, write-only */
    #define OVPHYSX_ATTR_THRUST1                  "thrust1"                  /* f32, [-1,1], tank-only, write-only */
    #define OVPHYSX_ATTR_CLUTCH                   "clutch"                   /* f32, [0,1], read-only */
    #define OVPHYSX_ATTR_CURRENT_GEAR             "currentGear"              /* int32, read-only */
    #define OVPHYSX_ATTR_TARGET_GEAR              "targetGear"               /* int32, internal target, read-only */
    #define OVPHYSX_ATTR_ENGINE_ROTATION_SPEED    "engineRotationSpeed"      /* f32, radians/s, read-only */
    #define OVPHYSX_ATTR_AUTOMATIC_TRANSMISSION   "automaticTransmission"    /* uint8, read-only */
    #define OVPHYSX_ATTR_GEAR_SWITCH_TIME         "gearSwitchTime"           /* f32 seconds, read-only; negative if no shift */
    #define OVPHYSX_ATTR_AUTOBOX_TIME_SINCE_LAST_SHIFT "autoboxTimeSinceLastShift" /* f32 seconds, read-only */
    /* One-time command at commit: 0 leaves the row unchanged, 1 invokes the existing rest reset.
       Every addressed chassis must have simulation enabled, including rows containing 0.
       Chassis pose and controls are preserved; internal dynamics and chassis velocities/forces
       are reset. Commit reset before writing desired velocities. No later step replays it. */
    #define OVPHYSX_ATTR_RESET_TO_REST_STATE      "resetToRestState"         /* uint8, write-only */

    /* Vehicle wheel controls: fixed CPU f32 columns for vehicles WITHOUT a drive. They persist
       until rewritten. Composed position/orientation and all telemetry below are read-only.
       steerAngle is the command; wheelSteerAngle reports the actual wheel steering angle. */
    #define OVPHYSX_ATTR_DRIVE_TORQUE             "driveTorque"              /* finite f32, torque */
    #define OVPHYSX_ATTR_BRAKE_TORQUE             "brakeTorque"              /* finite f32 >= 0, torque */
    #define OVPHYSX_ATTR_STEER_ANGLE              "steerAngle"               /* finite f32, radians */
    #define OVPHYSX_ATTR_ROTATION_SPEED           "rotationSpeed"            /* f32, radians/s */
    #define OVPHYSX_ATTR_ROTATION_ANGLE           "rotationAngle"            /* f32, radians */
    #define OVPHYSX_ATTR_WHEEL_STEER_ANGLE         "wheelSteerAngle"          /* f32, radians */
    #define OVPHYSX_ATTR_SUSPENSION_JOUNCE         "suspensionJounce"         /* f32, scene length */
    #define OVPHYSX_ATTR_SUSPENSION_FORCE          "suspensionForce"          /* vec3 f32, world force */
    #define OVPHYSX_ATTR_TIRE_FRICTION             "tireFriction"             /* f32, dimensionless */
    #define OVPHYSX_ATTR_TIRE_LONGITUDINAL_DIRECTION "tireLongitudinalDirection" /* vec3 f32, world */
    #define OVPHYSX_ATTR_TIRE_LATERAL_DIRECTION    "tireLateralDirection"     /* vec3 f32, world */
    #define OVPHYSX_ATTR_TIRE_LONGITUDINAL_SLIP    "tireLongitudinalSlip"     /* f32, ratio */
    #define OVPHYSX_ATTR_TIRE_LATERAL_SLIP         "tireLateralSlip"          /* f32, radians */
    #define OVPHYSX_ATTR_TIRE_FORCE                "tireForce"                /* vec3 f32, world force */
    /* Ground contact validity, not a physical airborne/sleep classification. Ground plane, hit
       and identities are zero when isOnGround is 0 (airborne, sleeping, disabled or stale sample).
       Identity columns carry interned prim-path handles from the attached Stage dictionary;
       0 also means no source prim is available, including a default material. */
    #define OVPHYSX_ATTR_IS_ON_GROUND              "isOnGround"              /* uint8, contact sample valid */
    #define OVPHYSX_ATTR_GROUND_PLANE              "groundPlane"             /* vec4 f32, world: n.dot(p)+d=0 */
    #define OVPHYSX_ATTR_GROUND_HIT_POSITION       "groundHitPosition"       /* vec3 f32, world scene length */
    #define OVPHYSX_ATTR_GROUND_ACTOR_PATH         "groundActorPath"         /* uint64 prim-path handle */
    #define OVPHYSX_ATTR_GROUND_SHAPE_PATH         "groundShapePath"         /* uint64 prim-path handle */
    #define OVPHYSX_ATTR_GROUND_MATERIAL_PATH      "groundMaterialPath"      /* uint64 prim-path handle */

    /* Character-controller movement. A commit calls PxController::move once with this
       world-space displacement and the scene timestep. Later steps do not replay it.
       Refused on a DirectGPU scene. Pose is the rigid-body position and orientation. */
    #define OVPHYSX_ATTR_CCT_MOVE               "move"                  /* vec3 f32 [conditional] */

    /* Debug render (OVPHYSX_QUERY_TARGET_DEBUG_RENDER). One instance-level target, not a prim.
       Settings are one row. Geometry columns are the primitive list for this instance's
       scenes: point [N,3] / uint32 [N], line [N,2,3] / uint32 [N,2], triangle [N,3,3] /
       uint32 [N,3]. Position index i matches color index i. N = 0 is an empty list.
       A scope read is uint64 path tokens, shape [N]. N = 0 draws every object.
       A scope write is uint64 shape [4097]: element 0 is the count (at most 4096)
       and elements 1..count are the tokens. A culling-box read is 6 f32
       (min xyz, max xyz), lanes = 6; leading shape 0 means no box. A culling-box
       write is f32 shape [7]: element 0 is 1 to store the next six floats, or 0 to clear.

       Each name below is also an ovx_string_or_token_t (OVPHYSX_NAME_DEBUG_*). Pass
       that object's address to query/read/write. token is 0, so the string is the name. */

    #define OVPHYSX_ATTR_DEBUG_ENABLED                  "enabled"
    #define OVPHYSX_ATTR_DEBUG_SCALE                    "scale"
    #define OVPHYSX_ATTR_DEBUG_WORLD_AXES               "worldAxes"
    #define OVPHYSX_ATTR_DEBUG_BODY_AXES                "bodyAxes"
    #define OVPHYSX_ATTR_DEBUG_BODY_MASS_AXES           "bodyMassAxes"
    #define OVPHYSX_ATTR_DEBUG_BODY_LINEAR_VELOCITY     "bodyLinearVelocity"
    #define OVPHYSX_ATTR_DEBUG_BODY_ANGULAR_VELOCITY    "bodyAngularVelocity"
    #define OVPHYSX_ATTR_DEBUG_CONTACT_POINT            "contactPoint"
    #define OVPHYSX_ATTR_DEBUG_CONTACT_NORMAL           "contactNormal"
    #define OVPHYSX_ATTR_DEBUG_CONTACT_ERROR            "contactError"
    #define OVPHYSX_ATTR_DEBUG_CONTACT_IMPULSE          "contactImpulse"
    #define OVPHYSX_ATTR_DEBUG_FRICTION_POINT           "frictionPoint"
    #define OVPHYSX_ATTR_DEBUG_FRICTION_NORMAL          "frictionNormal"
    #define OVPHYSX_ATTR_DEBUG_FRICTION_IMPULSE         "frictionImpulse"
    #define OVPHYSX_ATTR_DEBUG_ACTOR_AXES               "actorAxes"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_AABBS          "collisionAABBs"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_SHAPES         "collisionShapes"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_AXES           "collisionAxes"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_COMPOUNDS      "collisionCompounds"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_FACE_NORMALS   "collisionFaceNormals"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_EDGES          "collisionEdges"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_STATIC_PRUNER  "collisionStaticPruner"
    #define OVPHYSX_ATTR_DEBUG_COLLISION_DYNAMIC_PRUNER "collisionDynamicPruner"
    #define OVPHYSX_ATTR_DEBUG_JOINT_LOCAL_FRAMES       "jointLocalFrames"
    #define OVPHYSX_ATTR_DEBUG_JOINT_LIMITS             "jointLimits"
    #define OVPHYSX_ATTR_DEBUG_CULL_BOX                 "cullBox"
    #define OVPHYSX_ATTR_DEBUG_MBP_REGIONS              "mbpRegions"
    #define OVPHYSX_ATTR_DEBUG_SIMULATION_MESH          "simulationMesh"
    #define OVPHYSX_ATTR_DEBUG_SDF                      "sdf"
    #define OVPHYSX_ATTR_DEBUG_CULLING_BOX              "cullingBox"
    #define OVPHYSX_ATTR_DEBUG_SCOPE                    "scope"
    #define OVPHYSX_ATTR_DEBUG_POINT_POSITION           "pointPosition"
    #define OVPHYSX_ATTR_DEBUG_POINT_COLOR              "pointColor"
    #define OVPHYSX_ATTR_DEBUG_LINE_POSITION            "linePosition"
    #define OVPHYSX_ATTR_DEBUG_LINE_COLOR               "lineColor"
    #define OVPHYSX_ATTR_DEBUG_TRIANGLE_POSITION        "trianglePosition"
    #define OVPHYSX_ATTR_DEBUG_TRIANGLE_COLOR           "triangleColor"

#endif /* OVPHYSX_ATTRIBUTES_H */
