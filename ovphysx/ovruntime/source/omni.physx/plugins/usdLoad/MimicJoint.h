// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once


#include <private/omni/physx/PhysxUsd.h>

#include "AttachedStage.h"


namespace omni
{
namespace physx
{
namespace usdparser
{


// Mimic-joint parsing (PhysxMimicJointAPI + NewtonMimicAPI) lives in the parse
// library's native walker (`finalizeMimicJoints`); see `omni.physics.parse/ParseMimicJoint.cpp`
// and REQ-PARSE-MIMIC-001. Only the engine-side create/release helpers remain here.

ObjectId createMimicJoint(AttachedStage&, MimicJointDesc&);
void releaseMimicJoint(AttachedStage&, const PXR_NS::SdfPath& path, SchemaAPIFlag::Enum);


} // namespace usdparser
} // namespace physx
} // namespace omni
