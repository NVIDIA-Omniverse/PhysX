// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-DEBUG-001
// @covers AC-1 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/debug.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeDebug.h>
#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <cstring>
#include <limits>
namespace ovphysx::companion::debug
{
using namespace omni::physx::native;
using Status=ovphysx_companion_status_t;

Status convert(DebugResult r) {
    switch(r) {
    case DebugResult::Success:return OVPHYSX_COMPANION_OK;
    case DebugResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case DebugResult::Stale:return OVPHYSX_COMPANION_STALE;
    case DebugResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case DebugResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
    case DebugResult::Limit:return OVPHYSX_COMPANION_DEBUG_LIMIT;
    case DebugResult::Error:return OVPHYSX_COMPANION_ERROR;
    }
    return OVPHYSX_COMPANION_ERROR;
}
Status configure(uint64_t attachment, const ovphysx_companion_debug_config_t* input) noexcept {
    if (!input || input->size!=sizeof(*input) || input->version!=1 || input->enabled>1 || input->culling>1 || input->reserved)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());
    if (!guard) return OVPHYSX_COMPANION_BUSY;
    DebugConfig config;
    config.parameters=input->parameters; config.scale=input->scale;
    config.enabled=input->enabled!=0; config.culling=input->culling!=0;
    std::copy_n(input->minimum,3,config.minimum); std::copy_n(input->maximum,3,config.maximum);
    return convert(configureDebug(attachment,config));
}
Status capture(uint64_t attachment, uint64_t limit, uint64_t* output) noexcept {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        CompanionOperationGuard guard(companionRuntime());
        if (!guard) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<omni::physx::native::DebugFrame> frame=std::make_shared<DebugFrame>();
        const Status status=convert(captureDebug(attachment,limit,*frame));
        if (status!=OVPHYSX_COMPANION_OK) return status;
        std::lock_guard<std::mutex> lock(companionRuntime().debugFrames().mutex);
        if (companionRuntime().debugFrames().nextHandle==std::numeric_limits<uint64_t>::max()) return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle=companionRuntime().debugFrames().nextHandle++;
        companionRuntime().debugFrames().entries.emplace(handle,std::move(frame));
        *output=handle;
        return OVPHYSX_COMPANION_OK;
    } catch (...) {return OVPHYSX_COMPANION_ERROR;}
}
template<typename Fn> Status withFrame(uint64_t handle, Fn fn) noexcept {
    try {
        std::shared_ptr<const DebugFrame> frame;
        {std::lock_guard<std::mutex> lock(companionRuntime().debugFrames().mutex);
         std::map<uint64_t, std::shared_ptr<const omni::physx::native::DebugFrame>>::iterator it=companionRuntime().debugFrames().entries.find(handle); if (it==companionRuntime().debugFrames().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
         frame=it->second;}
        return fn(*frame);
    } catch (...) {return OVPHYSX_COMPANION_ERROR;}
}
Status describe(uint64_t handle,uint32_t size,ovphysx_companion_debug_info_t* output) noexcept {
    if (!output || size!=sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withFrame(handle,[&](const DebugFrame& frame)->Status {
        *output={sizeof(*output),1,frame.attachment,frame.step,frame.points.size(),frame.lines.size()/2,frame.triangles.size()/3};
        return OVPHYSX_COMPANION_OK;
    });
}
Status copy(uint64_t handle,uint32_t kind,ovphysx_companion_debug_vertex_t* data,uint64_t capacity,uint64_t* required) noexcept {
    if (!required || kind>2 || (!data && capacity)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withFrame(handle,[&](const DebugFrame& frame)->Status {
        const std::vector<DebugVertex> & vertices=kind==0?frame.points:kind==1?frame.lines:frame.triangles;
        *required=vertices.size();
        if (!data && !capacity) return OVPHYSX_COMPANION_OK;
        if (capacity<vertices.size()) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        static_assert(sizeof(DebugVertex)==sizeof(*data));
        if (!vertices.empty()) std::memcpy(data,vertices.data(),vertices.size()*sizeof(*data));
        return OVPHYSX_COMPANION_OK;
    });
}
Status release(uint64_t handle) noexcept {
    try {std::lock_guard<std::mutex> lock(companionRuntime().debugFrames().mutex);
         return companionRuntime().debugFrames().entries.erase(handle)?OVPHYSX_COMPANION_OK:OVPHYSX_COMPANION_NOT_FOUND;}
    catch (...) {return OVPHYSX_COMPANION_ERROR;}
}
}
using namespace ovphysx::companion::debug;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_debug_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_debug_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,1,omni::physx::native::debugParameterMask(),configure,capture,describe,copy,release};
    return OVPHYSX_COMPANION_OK;
}
