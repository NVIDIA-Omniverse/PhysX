// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-INTERACTION-001
// @covers AC-1 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-5
#include "ovphysx_companion/native_access.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeInteraction.h>
namespace sdk=ovphysx::companion::native;
namespace rt=omni::physx::native;
namespace ovphysx::companion::native
{
struct AccessLeaseFactory
{
    static bool invoke(const ovphysx_companion_object_t& ref,AccessKind kind,const void* ptr,AccessVisitor visitor,void* context) noexcept
    {
        ObjectLease lease(ref,kind,ptr);return visitor(lease,context);
    }
};
}
namespace
{
struct Context {ovphysx_companion_object_t ref;sdk::AccessVisitor callback;void* user;};
bool invoke(rt::LeaseKind kind,const void* pointer,void* raw) noexcept
{
    const Context & context=*static_cast<const Context*>(raw);
    CompanionNativeAccessScope scope(companionRuntime());
    return sdk::AccessLeaseFactory::invoke(context.ref,sdk::AccessKind(kind),pointer,context.callback,context.user);
}
ovphysx_companion_status_t visit(const ovphysx_companion_object_t* ref,uint32_t size,
    sdk::AccessKind kind,sdk::AccessVisitor callback,void* user) noexcept
{
    if(!ref || size!=sizeof(*ref) || !callback)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());if(!guard)return OVPHYSX_COMPANION_BUSY;
    Context context{*ref,callback,user};
    const rt::ObjectReference reference{ref->attachment,ref->object,ref->key,ref->generation,ref->kind,ref->instance};
    switch(rt::visitNativeObject(reference,rt::LeaseKind(kind),invoke,&context)) {
    case rt::InspectionResult::Success:return OVPHYSX_COMPANION_OK;
    case rt::InspectionResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case rt::InspectionResult::Stale:return OVPHYSX_COMPANION_STALE;
    case rt::InspectionResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case rt::InspectionResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
    default:return OVPHYSX_COMPANION_ERROR;
    }
}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_native_access_acquire(
    const sdk::Compatibility* input,uint32_t size,uint32_t outputSize,sdk::NativeAccessApi* output)
{
    if(!input || !output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if(size!=sizeof(*input) || outputSize!=sizeof(*output) || !sdk::compatibleBuild(*input))return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,1,visit};return OVPHYSX_COMPANION_OK;
}
