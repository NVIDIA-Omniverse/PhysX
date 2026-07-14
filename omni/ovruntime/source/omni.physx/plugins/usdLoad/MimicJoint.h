// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


//! Parse mimic joints authored with the PhysX-native PhysxMimicJointAPI (multi-applied per rotational axis;
//! natively supports multi-DOF joints) and append the resulting descriptors to @p mimicJointDescList.
void parseMimicJoints(const PXR_NS::UsdStageWeakPtr, const PXR_NS::UsdPrim&, std::vector<MimicJointDesc>& mimicJointDescList);

//! Parse mimic joints authored with the Newton single-apply NewtonMimicAPI (single-DOF joints only: revolute with
//! a finite limit or prismatic) and append the resulting descriptors to @p mimicJointDescList. Newton coefficients
//! are converted to the PhysX gearing/offset convention (sign-flipped) during parsing.
void parseNewtonMimicJoints(const PXR_NS::UsdStageWeakPtr, const PXR_NS::UsdPrim&, std::vector<MimicJointDesc>& mimicJointDescList);


ObjectId createMimicJoint(AttachedStage&, MimicJointDesc&);
void releaseMimicJoint(AttachedStage&, const PXR_NS::SdfPath& path, SchemaAPIFlag::Enum);


} // namespace usdparser
} // namespace physx
} // namespace omni
