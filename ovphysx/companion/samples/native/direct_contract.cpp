// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-5
#include "direct_custom.h"
#include <ovphysx_companion/events.hpp>
#include <type_traits>
static_assert(std::is_same_v<decltype(ovphysx::companion::native::CustomInterfaces::joints),
    omni::physx::IPhysxCustomJoint*>);
static_assert(std::is_same_v<decltype(ovphysx::companion::native::CustomInterfaces::geometry),
    omni::physx::IPhysxCustomGeometry*>);
int main()
{
    // This installed-header consumer has only the packaged SDK and PhysX include
    // directories. It links no runtime, Carbonite or USD library.
    const ovphysx::companion::native::Compatibility build;
    return ovphysx::companion::native::compatibleBuild(build) ? 0 : 1;
}
