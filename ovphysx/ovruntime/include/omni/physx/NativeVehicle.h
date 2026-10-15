// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-VEHICLE-CONTROL-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <cstdint>
namespace omni::physx::native
{
enum class VehicleResult : uint32_t { Success, Invalid, Stale, Busy, NotFound, Unavailable, Error };
struct VehicleIdentity
{
    uint64_t key, generation;
};
struct VehicleWheel
{
    VehicleIdentity identity, groundActor, groundShape, groundMaterial;
    uint32_t index, onGround, comFrame, reserved;
    float position[3], orientation[4], rotationSpeed, rotationAngle, steerAngle;
    float groundPlane[4], groundPosition[3], suspensionJounce, suspensionForce[3];
    float tireFriction, longitudinalDirection[3], lateralDirection[3], longitudinalSlip, lateralSlip, tireForce[3];
};
struct VehicleCommand
{
    uint64_t generation;
    uint32_t action; // 0 describe, 1 apply inputs, 2 clear inputs. Mutations require generation.
    int32_t direction; // -2 preserve transmission, -1 reverse, 0 neutral, 1 first forward.
    float throttle, brake0, brake1, steer;
};
struct VehicleState
{
    VehicleIdentity identity;
    uint64_t step;
    uint32_t engineFields, wheelCount;
    float throttle, brake0, brake1, steer, forwardVelocity, clutch, gearSwitchTime, autoboxTime, engineSpeed;
    int32_t currentGear, targetGear;
    uint32_t automaticTransmission, reserved;
    VehicleWheel wheels[20];
};
// Exact idle CPU attachment; ordinary managed controllers only (not tanks/raw
// wheels). No cached actor pointers. Controls do not wake/step or change USD.
// All values use stage length/mass units and seconds. Wheel pose frame is explicit.
// Record changes invalidate cached ground contacts until the next vehicle update.
// Rejected commands preserve output. Clear resets throttle/brakes/steer only.
VehicleResult controlVehicle(uint64_t attachment, const char* path, const VehicleCommand&, VehicleState&) noexcept;
// Stopped CPU computations: meter/kg data, quaternions XYZW, no SDK/owner setup.
// Bounded to 20 wheels. Invalid/ill-conditioned inputs preserve output.
bool vehicleSprungMasses(uint32_t count, const float* xyz, float mass, uint32_t gravityAxis, float* output) noexcept;
bool vehicleSuspensionFrame(const float* bodyPose, const float* wheelPose, float* output) noexcept;
bool vehicleAckermann(float wheelBase, float trackWidth, float innerAngle, float& outerAngle) noexcept;
}
