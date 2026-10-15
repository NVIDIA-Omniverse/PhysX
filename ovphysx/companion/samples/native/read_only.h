// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-INTERACTION-001
// @covers AC-1 AC-4
#pragma once
#include <ovphysx_companion/native_access.h>
namespace companion_sample
{
// Exceptional C++ SDK inspection: copy a value and return before the host steps.
// Extensions using Python should use Inspector.read instead of an SDK callback.
inline ovphysx_companion_status_t readNativeMass(
    const ovphysx::companion::native::NativeAccessApi& api,
    const ovphysx_companion_object_t& body,float& mass) noexcept
{
    using namespace ovphysx::companion::native;
    float copied=0;
    const ovphysx_companion_status_t status=api.visit(&body,sizeof(body),AccessKind::Dynamic,
        [](const ObjectLease& lease,void* destination) noexcept {
            const ::physx::PxRigidDynamic* dynamic=lease.dynamic();
            if(!dynamic)return false;
            *static_cast<float*>(destination)=dynamic->getMass();
            return true;
        },&copied);
    if(status==OVPHYSX_COMPANION_OK)mass=copied;
    return status;
}
}
