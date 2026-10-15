// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-QUERY-001
// @covers AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_INSPECTION_H
#define OVPHYSX_COMPANION_INSPECTION_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
#define OVPHYSX_COMPANION_INSPECTION_CAPACITY 8u
typedef struct ovphysx_companion_object {
    uint64_t attachment, object, key, generation;
    uint32_t kind, instance;
} ovphysx_companion_object_t;
typedef struct ovphysx_companion_body {
    ovphysx_companion_object_t identity;
    uint64_t step;
    uint32_t type, flags, locks, reserved;
    float pose[7], centerOfMass[7], mass, inertia[3], linearVelocity[3], angularVelocity[3];
} ovphysx_companion_body_t;
typedef struct ovphysx_companion_joint {
    ovphysx_companion_object_t identity, body0, body1;
    uint64_t step;
    uint32_t type, flags, validFields, reserved;
    float local0[7], local1[7], world0[7], world1[7], breakForce, breakTorque;
} ovphysx_companion_joint_t;
typedef struct ovphysx_companion_material {
    ovphysx_companion_object_t identity;
    uint64_t step;
    float staticFriction, dynamicFriction, restitution;
    uint32_t frictionCombine, restitutionCombine, flags;
} ovphysx_companion_material_t;
typedef struct ovphysx_companion_dof {
    ovphysx_companion_object_t link, joint;
    uint32_t index, axis;
    float position, velocity, acceleration, force;
} ovphysx_companion_dof_t;
typedef struct ovphysx_companion_articulation {
    ovphysx_companion_object_t identity;
    uint64_t step;
    uint32_t flags, sleeping;
    float rootPose[7], rootLinearVelocity[3], rootAngularVelocity[3];
} ovphysx_companion_articulation_t;
/** Copied CPU state, idle exact-owner admission; retain an owner lease during calls.
 * resolve kinds: 1 body, 2 joint, 3 material, 4 articulation, 5 shape.
 * instance UINT32_MAX selects an uninstanced object. IDs are records, never pointers.
 * Read kind is reference.kind (1..3); output size must exactly match that struct.
 * Articulation arrays and summary come from one admitted read. Capacity status 8
 * changes only requiredLinks/requiredDofs; NULL/zero buffers perform sizing.
 * Other failures preserve all outputs. Sizes/versions negotiate exact ABI layout.
 * Units: scene length/mass, seconds, radians. Pose = xyz + quaternion xyzw.
 * Body pose and joint world frames are world; COM and joint local frames actor-local.
 * Static body mass/inertia/velocity fields are not applicable and zero.
 * Body type: 0 static, 1 dynamic, 2 link. Flags: 1 kinematic, 2 sleeping,
 * 4 CCD, 8 simulation disabled. Locks are PxRigidDynamicLockFlag bits.
 * Joint type: 0 generic, 1 fixed, 2 revolute, 3 prismatic, 4 spherical,
 * 5 distance, 6 D6, 7 articulation. validFields bit0 validates SDK flags and
 * break thresholds (ordinary joints only). Zero body reference means world.
 * Material combine enums and flags use PxCombineMode/PxMaterialFlag.
 * Articulation links use SDK link-index order; DOFs use cache order and axes
 * twist/swing1/swing2/x/y/z (0..5). Force is applied joint force, not measured
 * constraint force. Results remain valid copies after owner retirement.
 * GPU live body/joint/articulation reads are unavailable; material is host metadata.
 */
typedef struct ovphysx_companion_inspection_api {
    uint32_t size, version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*resolve)(uint64_t attachment,const char* path,uint32_t kind,uint32_t instance,
        uint32_t size,ovphysx_companion_object_t* output);
    ovphysx_companion_status_t (*read)(const ovphysx_companion_object_t*,uint32_t referenceSize,void* output,uint32_t outputSize);
    ovphysx_companion_status_t (*articulation)(const ovphysx_companion_object_t*,uint32_t referenceSize,
        ovphysx_companion_articulation_t*,uint32_t outputSize,
        ovphysx_companion_body_t* links,uint32_t linkCapacity,uint32_t* requiredLinks,
        ovphysx_companion_dof_t* dofs,uint32_t dofCapacity,uint32_t* requiredDofs);
    /* UTF-8 diagnostic authored path; required includes NUL. Capacity status 8
     * only writes required; NULL/0 sizes. Identity is validated on each call. */
    ovphysx_companion_status_t (*path)(const ovphysx_companion_object_t*,uint32_t referenceSize,
        char* output,uint32_t capacity,uint32_t* required);
} ovphysx_companion_inspection_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_inspection_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_inspection_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
