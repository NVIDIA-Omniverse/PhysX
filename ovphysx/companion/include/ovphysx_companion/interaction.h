// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-INTERACTION-001
// @covers AC-2 AC-3 AC-4 AC-5 AC-6
#ifndef OVPHYSX_COMPANION_INTERACTION_H
#define OVPHYSX_COMPANION_INTERACTION_H
#include "inspection.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct ovphysx_companion_force {
    uint32_t mode, frame, atPoint, wake;
    float force[3], point[3];
} ovphysx_companion_force_t;
typedef struct ovphysx_companion_drag_options {
    float target[3], stiffness, damping, maxForce;
    uint32_t flags, velocityPolicy;
} ovphysx_companion_drag_options_t;
typedef struct ovphysx_companion_drag_state {
    ovphysx_companion_object_t body;
    float target[3];
    uint32_t active;
} ovphysx_companion_drag_state_t;
/** CPU movable dynamic bodies, exact live record identity, idle admission.
 * Force: mode 0 force, 1 impulse; frame 0 world, 1 actor-local; atPoint and wake
 * are bool. Point transforms from actor-local if requested; absent point means
 * COM. Units: scene mass*length/seconds^2 or mass*length/seconds. No implicit step.
 * Static, kinematic, articulation links, disabled and GPU actors are unavailable.
 * Drag: target is the initial picked world point, later updated in world units.
 * D6 spring drives that local body point, with force stiffness/damping/limit.
 * flags bit0 temporarily clears axis locks; bit1 enables speculative CCD. End
 * restores those fields; no scene settings change. velocityPolicy: 0 preserve
 * final velocities, 1 zero, 2 restore initial (only while still movable).
 * One live drag per body. close is retryable on BUSY; deletion/reset closes the
 * native constraint before actor destruction. A retired drag reads active=0;
 * close then releases its runtime lease. Handles are process-local and never reused.
 * No interpreter callbacks or raw SDK pointers. All sizes must exactly match.
 * Failed calls preserve outputs. Callers retain the owner during force/begin;
 * a successful drag owns an independent service lease until close.
 */
typedef struct ovphysx_companion_interaction_api {
    uint32_t size,version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*force)(const ovphysx_companion_object_t*,uint32_t referenceSize,
        const ovphysx_companion_force_t*,uint32_t commandSize);
    ovphysx_companion_status_t (*begin)(const ovphysx_companion_object_t*,uint32_t referenceSize,
        const ovphysx_companion_drag_options_t*,uint32_t optionsSize,uint64_t* drag);
    ovphysx_companion_status_t (*move)(uint64_t drag,const float target[3],
        ovphysx_companion_drag_state_t*,uint32_t stateSize);
    ovphysx_companion_status_t (*read)(uint64_t drag,ovphysx_companion_drag_state_t*,uint32_t stateSize);
    ovphysx_companion_status_t (*close)(uint64_t drag);
} ovphysx_companion_interaction_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_interaction_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_interaction_api_t* output);
/** Managed force picking. Separate versioned table preserves the original D6
 * interaction ABI. begin captures the world picked point, actor-local anchor and
 * original ray distance. pickingForce >= 0 scales the legacy coefficient k=50*f;
 * pickDistance is in [0,1000000] scene length units. Default end velocity preserves
 * motion; policies 1 zero / 2 restore are supported on dynamic rigid bodies only.
 * CPU articulation links support policy 0. Static/kinematic/disabled/GPU bodies
 * return UNAVAILABLE. No joint, axis-lock or CCD edits are made.
 * Each actual native scene substep applies J=k*m*(target-p-0.02*vPoint)*h at p,
 * plus legacy COM gravity assistance. Native code restores angular damping and
 * wakes movable dynamic bodies on close. No forces are applied by move/read or
 * during a frame with no due physics substep. Release/cancellation needs no step.
 * move_ray normalizes direction and retains the initial distance; move accepts an
 * explicit world target. One joint or force grab may own a body at a time.
 * Host input filters/timeline/focus events remain host-owned: close on cancel,
 * Pause/Stop/focus loss. Deletion/detach automatically retire native state.
 * All sizes must match; failed calls preserve outputs. Handles are compatible
 * with the original interaction table's move/read/close. Caller retains the owner
 * for begin; the grab owns a runtime service lease until close.
 */
typedef struct ovphysx_companion_picking_options {
    float point[3], pickingForce, pickDistance;
    uint32_t velocityPolicy;
} ovphysx_companion_picking_options_t;
typedef struct ovphysx_companion_picking_api {
    uint32_t size,version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*begin)(const ovphysx_companion_object_t*,uint32_t referenceSize,
        const ovphysx_companion_picking_options_t*,uint32_t optionsSize,uint64_t* grab);
    ovphysx_companion_status_t (*move)(uint64_t grab,const float target[3],
        ovphysx_companion_drag_state_t*,uint32_t stateSize);
    ovphysx_companion_status_t (*read)(uint64_t grab,ovphysx_companion_drag_state_t*,uint32_t stateSize);
    ovphysx_companion_status_t (*close)(uint64_t grab);
    ovphysx_companion_status_t (*move_ray)(uint64_t grab,const float origin[3],const float direction[3],
        ovphysx_companion_drag_state_t*,uint32_t stateSize);
    /** Change the per-grab coefficient while idle, including zero. The next
     * substep uses it; angular damping retains the initial grab value. */
    ovphysx_companion_status_t (*set_force_coefficient)(uint64_t grab,float coefficient);
} ovphysx_companion_picking_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_picking_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_picking_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
