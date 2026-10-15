// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-VEHICLE-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/vehicle.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeVehicle.h>
#include <algorithm>
namespace
{
using namespace omni::physx::native;
ovphysx_companion_vehicle_identity_t convert(const VehicleIdentity& v) { return {v.key,v.generation}; }
ovphysx_companion_vehicle_wheel_t convert(const VehicleWheel& v)
{
    ovphysx_companion_vehicle_wheel_t o{};
    o.identity=convert(v.identity); o.groundActor=convert(v.groundActor);
    o.groundShape=convert(v.groundShape); o.groundMaterial=convert(v.groundMaterial);
    o.index=v.index; o.onGround=v.onGround; o.comFrame=v.comFrame;
    std::copy_n(v.position,3,o.position); std::copy_n(v.orientation,4,o.orientation);
    o.rotationSpeed=v.rotationSpeed; o.rotationAngle=v.rotationAngle; o.steerAngle=v.steerAngle;
    std::copy_n(v.groundPlane,4,o.groundPlane); std::copy_n(v.groundPosition,3,o.groundPosition);
    o.suspensionJounce=v.suspensionJounce; std::copy_n(v.suspensionForce,3,o.suspensionForce);
    o.tireFriction=v.tireFriction; std::copy_n(v.longitudinalDirection,3,o.longitudinalDirection);
    std::copy_n(v.lateralDirection,3,o.lateralDirection); o.longitudinalSlip=v.longitudinalSlip;
    o.lateralSlip=v.lateralSlip; std::copy_n(v.tireForce,3,o.tireForce); return o;
}
ovphysx_companion_status_t execute(uint64_t attach, const char* path,
    const ovphysx_companion_vehicle_command_t* input, uint32_t inputSize,
    ovphysx_companion_vehicle_state_t* output, uint32_t outputSize) noexcept
{
    if (!input || !output || inputSize!=sizeof(*input) || outputSize!=sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());
    if (!guard) return OVPHYSX_COMPANION_BUSY;
    const VehicleCommand cmd{input->generation,input->action,input->direction,input->throttle,input->brake0,input->brake1,input->steer};
    VehicleState state{};
    switch (controlVehicle(attach,path,cmd,state))
    {
    case VehicleResult::Invalid: return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case VehicleResult::Stale: return OVPHYSX_COMPANION_STALE;
    case VehicleResult::Busy: return OVPHYSX_COMPANION_BUSY;
    case VehicleResult::NotFound: return OVPHYSX_COMPANION_NOT_FOUND;
    case VehicleResult::Unavailable: return OVPHYSX_COMPANION_UNAVAILABLE;
    case VehicleResult::Error: return OVPHYSX_COMPANION_ERROR;
    case VehicleResult::Success: break;
    }
    ovphysx_companion_vehicle_state_t value{};
    value.identity=convert(state.identity); value.step=state.step; value.engineFields=state.engineFields;
    value.wheelCount=state.wheelCount; value.throttle=state.throttle; value.brake0=state.brake0;
    value.brake1=state.brake1; value.steer=state.steer; value.forwardVelocity=state.forwardVelocity;
    value.clutch=state.clutch; value.gearSwitchTime=state.gearSwitchTime; value.autoboxTime=state.autoboxTime;
    value.engineSpeed=state.engineSpeed; value.currentGear=state.currentGear; value.targetGear=state.targetGear;
    value.automaticTransmission=state.automaticTransmission;
    for (uint32_t i=0;i<state.wheelCount;++i) value.wheels[i]=convert(state.wheels[i]);
    *output=value; return OVPHYSX_COMPANION_OK;
}
ovphysx_companion_status_t masses(uint32_t count,const float* xyz,float mass,uint32_t axis,float* out,uint32_t capacity) noexcept
{
    return capacity>=count && vehicleSprungMasses(count,xyz,mass,axis,out) ? OVPHYSX_COMPANION_OK : OVPHYSX_COMPANION_INVALID_ARGUMENT;
}
ovphysx_companion_status_t frame(const float* body,const float* wheel,float* out) noexcept
{
    return vehicleSuspensionFrame(body,wheel,out) ? OVPHYSX_COMPANION_OK : OVPHYSX_COMPANION_INVALID_ARGUMENT;
}
ovphysx_companion_status_t steer(float base,float width,float inner,float* out) noexcept
{
    return out && vehicleAckermann(base,width,inner,*out) ? OVPHYSX_COMPANION_OK : OVPHYSX_COMPANION_INVALID_ARGUMENT;
}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_vehicle_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_vehicle_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,3,execute,masses,frame,steer}; return OVPHYSX_COMPANION_OK;
}
