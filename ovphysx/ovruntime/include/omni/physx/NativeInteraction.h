// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-INTERACTION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
#pragma once
#include "NativeInspection.h"
#include <memory>
namespace omni::physx::native
{
enum class LeaseKind:uint32_t {RigidActor=1,Dynamic=2,Static=3,Link=4,Joint=5,Articulation=6,Shape=7,Material=8};
using NativeVisitor=bool(*)(LeaseKind,const void*,void*) noexcept;
// Synchronous read-only, exact-SDK-type callback. Pointer is borrowed only for the
// duration of the callback; do not retain it or release/mutate its SDK object.
// No Python callback, nested owner operation, wait or lifecycle operation allowed.
InspectionResult visitNativeObject(const ObjectReference&,LeaseKind,NativeVisitor,void*) noexcept;
struct BodyForce
{
    uint32_t mode=0,frame=0,atPoint=0,wake=1; // force/impulse 0/1; world/actor-local 0/1
    float force[3]{},point[3]{};
};
// CPU dynamic rigid actors, idle owner. Kinematic/static/link/disabled bodies are
// unavailable. Point is absolute world or actor-local according to frame; absent
// point applies at COM. Units force=m*l/s^2, impulse=m*l/s. No implicit step/fetch.
InspectionResult applyBodyForce(const ObjectReference&,const BodyForce&) noexcept;
struct DragOptions
{
    float target[3]{},stiffness=1000,damping=100,maxForce=100000;
    uint32_t flags=3,velocityPolicy=0; // bit0 clear locks, bit1 speculative CCD
    // End velocity: 0 preserve final, 1 zero, 2 restore begin velocities.
};
struct DragState
{
    ObjectReference body;
    float target[3]{};
    uint32_t active=0;
};
struct ForceDragOptions
{
    float point[3]{},pickingForce=1,pickDistance=0;
    uint32_t velocityPolicy=0;
};
class Drag;
// Managed CPU force grab: retain the actor-local picked point and original ray
// distance. Native stepping applies the legacy impulse law once per actual scene
// substep. No joint, axis-lock change or CCD change. Angular damping restores on
// close; links support the preserve-velocity policy only. No implicit step/fetch.
InspectionResult beginForceDrag(const ObjectReference&,const ForceDragOptions&,std::shared_ptr<Drag>&) noexcept;
InspectionResult setForceDragCoefficient(const std::shared_ptr<Drag>&,float coefficient) noexcept;
InspectionResult moveDragRay(const std::shared_ptr<Drag>&,const float origin[3],const float direction[3],DragState&) noexcept;
// Collision-aware D6 spring at a body-local picked point. One drag per live body.
// End/owner reset/deletion releases the constraint before actor destruction.
// Only the temporarily changed locks/speculative-CCD bit are restored; scene
// settings are never changed. All close/update paths require idle owner admission;
// already invalidated sessions close successfully without touching a replacement.
InspectionResult beginDrag(const ObjectReference&,const DragOptions&,std::shared_ptr<Drag>&) noexcept;
InspectionResult moveDrag(const std::shared_ptr<Drag>&,const float target[3],DragState&) noexcept;
InspectionResult inspectDrag(const std::shared_ptr<Drag>&,DragState&) noexcept;
InspectionResult closeDrag(const std::shared_ptr<Drag>&) noexcept;
}
