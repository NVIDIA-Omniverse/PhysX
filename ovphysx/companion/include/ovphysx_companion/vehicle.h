// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-VEHICLE-001
// @covers AC-1 AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_VEHICLE_H
#define OVPHYSX_COMPANION_VEHICLE_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct ovphysx_companion_vehicle_identity {
    uint64_t key, generation;
} ovphysx_companion_vehicle_identity_t;
typedef struct ovphysx_companion_vehicle_wheel {
    ovphysx_companion_vehicle_identity_t identity, groundActor, groundShape, groundMaterial;
    uint32_t index, onGround, comFrame, reserved;
    float position[3], orientation[4], rotationSpeed, rotationAngle, steerAngle;
    float groundPlane[4], groundPosition[3], suspensionJounce, suspensionForce[3];
    float tireFriction, longitudinalDirection[3], lateralDirection[3], longitudinalSlip, lateralSlip, tireForce[3];
} ovphysx_companion_vehicle_wheel_t;
typedef struct ovphysx_companion_vehicle_command {
    uint64_t generation;
    uint32_t action; // 0 describe, 1 apply inputs, 2 clear inputs. Mutations require generation.
    int32_t direction; // -2 preserve transmission, -1 reverse, 0 neutral, 1 first forward.
    float throttle, brake0, brake1, steer;
} ovphysx_companion_vehicle_command_t;
typedef struct ovphysx_companion_vehicle_state {
    ovphysx_companion_vehicle_identity_t identity;
    uint64_t step;
    uint32_t engineFields, wheelCount;
    float throttle, brake0, brake1, steer, forwardVelocity, clutch, gearSwitchTime, autoboxTime, engineSpeed;
    int32_t currentGear, targetGear;
    uint32_t automaticTransmission, reserved;
    ovphysx_companion_vehicle_wheel_t wheels[20];
} ovphysx_companion_vehicle_state_t;
/** CPU managed vehicles; hold the selected owner lease through every call.
 * Stage length/mass units, seconds, angles in radians; quaternion XYZW.
 * execute requires idle admission; host invokes inputs before its declared step.
 * Generation zero is only for initial Describe. Clear leaves gear/pose/velocity.
 * engineFields=0 means clutch/gears/timers/engine speed/transmission unavailable.
 * Ground identities and plane/position are valid only when onGround;
 * after any native record change they are unavailable until the next vehicle update; zero
 * identity means no mapped source object. Identities are qualified by attachment.
 * wheel comFrame chooses scaled COM-local (1) or actor-local (0) poses.
 * Raw-wheel and tank controllers and GPU simulation are unavailable.
 * All failed calls preserve output. Computations do not require a live owner:
 * sprung masses use COM-relative meters, kg, gravity axis 0/1/2, max 20 wheels;
 * suspension frame transforms wheel world pose into supplied body/COM world frame;
 * Ackermann maps inner steer angle (-pi/2,pi/2) to outer angle for positive dimensions.
 */
typedef struct ovphysx_companion_vehicle_api {
    uint32_t size, version;
    uint64_t capabilities; /* bit0 CPU managed control/telemetry; bit1 stopped math */
    ovphysx_companion_status_t (*execute)(uint64_t attachment, const char* path,
        const ovphysx_companion_vehicle_command_t*, uint32_t command_size,
        ovphysx_companion_vehicle_state_t*, uint32_t output_size);
    ovphysx_companion_status_t (*sprung_masses)(uint32_t count, const float* xyz, float mass,
        uint32_t gravity_axis, float* output, uint32_t capacity);
    ovphysx_companion_status_t (*suspension_frame)(const float* body_pose, const float* wheel_pose, float* output);
    ovphysx_companion_status_t (*ackermann)(float wheel_base, float track_width, float inner_angle, float* outer_angle);
} ovphysx_companion_vehicle_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_vehicle_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_vehicle_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
