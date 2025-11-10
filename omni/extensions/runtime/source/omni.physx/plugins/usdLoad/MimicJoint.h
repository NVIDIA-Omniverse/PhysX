// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once


#include <private/omni/physx/PhysxUsd.h>

#include "AttachedStage.h"


namespace omni
{
namespace physx
{
namespace usdparser
{


void parseMimicJoints(const pxr::UsdStageWeakPtr, const pxr::UsdPrim&, std::vector<MimicJointDesc>& mimicJointDescList);


ObjectId createMimicJoint(AttachedStage&, MimicJointDesc&);
void releaseMimicJoint(AttachedStage&, const pxr::SdfPath& path, SchemaAPIFlag::Enum);


} // namespace usdparser
} // namespace physx
} // namespace omni
