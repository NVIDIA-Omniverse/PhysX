// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <private/omni/physx/PhysxUsd.h>

using namespace PXR_NS;

namespace omni
{
namespace physx
{
namespace usdparser
{

// Spatial tendon attachment parsing lives in the parse library (see
// `omni.physics.parse/ParseSpatialTendon.cpp` and REQ-PARSE-TENDON-001) and
// the native walker's `finalizeSpatialTendons` pass. Only the engine-side
// create helper remains here.

void createSpatialTendons(AttachedStage& attachedStage,
                          TendonAttachmentMap& attachmentMap,
                          SpatialTendonVector& spatialTendons);
} // namespace usdparser
} // namespace physx
} // namespace omni
