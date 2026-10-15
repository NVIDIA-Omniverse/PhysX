// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "read_only.h"
#include <ovphysx_companion/native_exports.h>
// The host negotiates NativeAccessApi using Compatibility from this exact SDK.
// No runtime or SDK static library is linked into this consumer.
OVPHYSX_COMPANION_PLUGIN_EXPORT ovphysx_companion_status_t companion_sample_read_mass(
    const ovphysx::companion::native::NativeAccessApi* api,
    const ovphysx_companion_object_t* body,float* mass) noexcept
{
    if(!api || !body || !mass)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return companion_sample::readNativeMass(*api,*body,*mass);
}
