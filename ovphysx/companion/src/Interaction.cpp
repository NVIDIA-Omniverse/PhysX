// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-INTERACTION-001
// @covers AC-2 AC-3 AC-4 AC-5 AC-6
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-4
#include "ovphysx_companion/interaction.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include "CarboniteLoader/CarboniteLoader.hpp"
#include <omni/physx/NativeInteraction.h>
#include <algorithm>
#include <cstring>
#include <map>
#include <mutex>
namespace ovphysx::companion::interaction
{
using namespace omni::physx::native;
using Status=ovphysx_companion_status_t;
ObjectReference reference(const ovphysx_companion_object_t& r)
{return {r.attachment,r.object,r.key,r.generation,r.kind,r.instance};}
Status convert(InspectionResult result)
{
    switch(result) {
    case InspectionResult::Success:return OVPHYSX_COMPANION_OK;
    case InspectionResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case InspectionResult::Stale:return OVPHYSX_COMPANION_STALE;
    case InspectionResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case InspectionResult::NotFound:return OVPHYSX_COMPANION_NOT_FOUND;
    case InspectionResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
    default:return OVPHYSX_COMPANION_ERROR;
    }
}
struct Session
{
    CompanionServiceLease lease{companionRuntime()};
    std::shared_ptr<ovphysx::CarboniteLoader> runtime;
    std::shared_ptr<Drag> drag;
    std::mutex mutex;
    bool closed=false;
    explicit Session(std::shared_ptr<ovphysx::CarboniteLoader> r):runtime(std::move(r)){}
    ~Session(){if(drag)closeDrag(drag);drag.reset();runtime.reset();}
};

Status force(const ovphysx_companion_object_t* ref,uint32_t size,const ovphysx_companion_force_t* input,uint32_t inputSize) noexcept
{
    if(!ref || size!=sizeof(*ref) || !input || inputSize!=sizeof(*input))return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard admission(companionRuntime());if(!admission)return OVPHYSX_COMPANION_BUSY;
    BodyForce command;command.mode=input->mode;command.frame=input->frame;command.atPoint=input->atPoint;command.wake=input->wake;
    std::copy_n(input->force,3,command.force);std::copy_n(input->point,3,command.point);
    return convert(applyBodyForce(reference(*ref),command));
}
Status begin(const ovphysx_companion_object_t* ref,uint32_t size,const ovphysx_companion_drag_options_t* input,
             uint32_t inputSize,uint64_t* output) noexcept
try
{
    if(!ref || size!=sizeof(*ref) || !input || inputSize!=sizeof(*input) || !output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard admission(companionRuntime());if(!admission)return OVPHYSX_COMPANION_BUSY;
    std::shared_ptr<ovphysx::CarboniteLoader> runtime=companionRuntimeForAttachment(ref->attachment);if(!runtime)return OVPHYSX_COMPANION_STALE;
    DragOptions options;std::copy_n(input->target,3,options.target);options.stiffness=input->stiffness;
    options.damping=input->damping;options.maxForce=input->maxForce;options.flags=input->flags;options.velocityPolicy=input->velocityPolicy;
    std::shared_ptr<Session> session=std::make_shared<Session>(std::move(runtime));
    // Allocate the registry node before creating a joint or changing actor state.
    // The native path cannot call this registry; all other operations release the
    // registry mutex before taking owner admission.
    std::lock_guard<std::mutex> lock(companionRuntime().interactions().mutex);
    if(companionRuntime().interactions().nextHandle==UINT64_MAX)return OVPHYSX_COMPANION_ERROR;
    const uint64_t handle=companionRuntime().interactions().nextHandle++;companionRuntime().interactions().entries.emplace(handle,session);
    const Status result=convert(beginDrag(reference(*ref),options,session->drag));
    if(result!=OVPHYSX_COMPANION_OK){companionRuntime().interactions().entries.erase(handle);return result;}
    *output=handle;return OVPHYSX_COMPANION_OK;
}
catch(...){return OVPHYSX_COMPANION_ERROR;}
Status beginPicking(const ovphysx_companion_object_t* ref,uint32_t size,const ovphysx_companion_picking_options_t* input,
                    uint32_t inputSize,uint64_t* output) noexcept
try
{
    if(!ref || size!=sizeof(*ref) || !input || inputSize!=sizeof(*input) || !output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard admission(companionRuntime());if(!admission)return OVPHYSX_COMPANION_BUSY;
    std::shared_ptr<ovphysx::CarboniteLoader> runtime=companionRuntimeForAttachment(ref->attachment);if(!runtime)return OVPHYSX_COMPANION_STALE;
    ForceDragOptions options;std::copy_n(input->point,3,options.point);options.pickingForce=input->pickingForce;
    options.pickDistance=input->pickDistance;options.velocityPolicy=input->velocityPolicy;
    std::shared_ptr<Session> session=std::make_shared<Session>(std::move(runtime));
    std::lock_guard<std::mutex> lock(companionRuntime().interactions().mutex);
    if(companionRuntime().interactions().nextHandle==UINT64_MAX)return OVPHYSX_COMPANION_ERROR;
    const uint64_t handle=companionRuntime().interactions().nextHandle++;companionRuntime().interactions().entries.emplace(handle,session);
    const Status result=convert(beginForceDrag(reference(*ref),options,session->drag));
    if(result!=OVPHYSX_COMPANION_OK){companionRuntime().interactions().entries.erase(handle);return result;}
    *output=handle;return OVPHYSX_COMPANION_OK;
}
catch(...){return OVPHYSX_COMPANION_ERROR;}
template<class Fn> Status withSession(uint64_t handle,Fn fn) noexcept
try
{
    CompanionOperationGuard admission(companionRuntime());if(!admission)return OVPHYSX_COMPANION_BUSY;
    std::shared_ptr<Session> session;
    {std::lock_guard<std::mutex> lock(companionRuntime().interactions().mutex);std::map<uint64_t, std::shared_ptr<Session>>::iterator found=companionRuntime().interactions().entries.find(handle);
     if(found==companionRuntime().interactions().entries.end())return OVPHYSX_COMPANION_NOT_FOUND;
     session=found->second;}
    std::unique_lock<std::mutex> lock(session->mutex,std::try_to_lock);
    if(!lock.owns_lock())return OVPHYSX_COMPANION_BUSY;
    if(session->closed)return OVPHYSX_COMPANION_NOT_FOUND;
    return fn(*session);
}
catch(...){return OVPHYSX_COMPANION_ERROR;}
Status read(uint64_t handle,ovphysx_companion_drag_state_t* output,uint32_t size) noexcept
{
    if(!output || size!=sizeof(*output))return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withSession(handle,[&](Session& session)->Status {
        DragState value;const Status status=convert(inspectDrag(session.drag,value));
        static_assert(sizeof(value)==sizeof(*output));
        if(status==OVPHYSX_COMPANION_OK)std::memcpy(output,&value,sizeof(value));
        return status;
    });
}
Status move(uint64_t handle,const float* target,ovphysx_companion_drag_state_t* output,uint32_t size) noexcept
{
    if(!output || size!=sizeof(*output) || !target)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withSession(handle,[&](Session& session)->Status {
        DragState value;const Status status=convert(moveDrag(session.drag,target,value));
        if(status==OVPHYSX_COMPANION_OK)std::memcpy(output,&value,sizeof(value));
        return status;
    });
}
Status moveRay(uint64_t handle,const float* origin,const float* direction,ovphysx_companion_drag_state_t* output,uint32_t size) noexcept
{
    if(!output || size!=sizeof(*output) || !origin || !direction)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withSession(handle,[&](Session& session)->Status {
        DragState value;const Status status=convert(moveDragRay(session.drag,origin,direction,value));
        if(status==OVPHYSX_COMPANION_OK)std::memcpy(output,&value,sizeof(value));
        return status;
    });
}
Status setPickingCoefficient(uint64_t handle,float coefficient) noexcept
{
    return withSession(handle,[&](Session& session)->Status {
        return convert(setForceDragCoefficient(session.drag,coefficient));
    });
}
Status close(uint64_t handle) noexcept
{
    return withSession(handle,[&](Session& session)->Status {
        const Status status=convert(closeDrag(session.drag));if(status!=OVPHYSX_COMPANION_OK)return status;
        session.closed=true;std::lock_guard<std::mutex> lock(companionRuntime().interactions().mutex);companionRuntime().interactions().entries.erase(handle);return OVPHYSX_COMPANION_OK;
    });
}
}
using namespace ovphysx::companion::interaction;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_interaction_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_interaction_api_t* output)
{
    if(!output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if(version!=1 || size!=sizeof(*output))return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,3,force,begin,move,read,close};return OVPHYSX_COMPANION_OK;
}

extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_picking_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_picking_api_t* output)
{
    if(!output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if(version!=1 || size!=sizeof(*output))return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,1,beginPicking,move,read,close,moveRay,setPickingCoefficient};return OVPHYSX_COMPANION_OK;
}
