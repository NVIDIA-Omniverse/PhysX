// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-5
// @implements REQ-COMP-JOINT-001
// @covers AC-5
// @implements REQ-COMP-GEOMETRY-001
// @covers AC-5
#pragma once
#include <ovphysx_companion/custom_interfaces.h>

namespace companion_sample
{
// The application owns callback code and per-object state. Retain the runtime
// lease and these registrations while any attached scene uses the custom types.
// This sample consumes the existing runtime callback structures unchanged.
struct CustomRegistrations
{
    size_t joint = 0;
    size_t geometry = 0;
    bool retire(const ovphysx::companion::native::CustomInterfaces& interfaces)
    {
        if (joint && interfaces.joints->tryUnregisterCustomJoint(joint))
            joint = 0;
        if (geometry && interfaces.geometry->tryUnregisterCustomGeometry(geometry))
            geometry = 0;
        return !joint && !geometry;
    }
};
inline CustomRegistrations registerCustomTypes(
    const ovphysx::companion::native::CustomInterfaces& interfaces,
    const char* jointType, omni::physx::ICustomJointCallback& jointCallbacks,
    physx::PxConstraintSolverPrep solver, size_t constantBlockSize,
    const char* geometryApi, omni::physx::ICustomGeometryCallback& geometryCallbacks)
{
    CustomRegistrations registrations;
    registrations.joint = interfaces.joints->registerCustomJoint(jointType, jointCallbacks, solver, constantBlockSize);
    registrations.geometry = interfaces.geometry->registerCustomGeometry(geometryApi, geometryCallbacks);
    return registrations;
}
}
