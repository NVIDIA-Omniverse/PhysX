// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

using namespace pxr;

namespace omni
{
namespace physx
{
namespace usdparser
{

void parseTendonAttachments(AttachedStage& attachedStage,
                            const UsdPrim& prim,
                            UsdGeomXformCache& xfCache,
                            TendonAttachmentMap& attachmentMap,
                            SpatialTendonVector& spatialTendons);

void createSpatialTendons(AttachedStage& attachedStage,
                          TendonAttachmentMap& attachmentMap,
                          SpatialTendonVector& spatialTendons);

std::vector<PhysxTendonAttachmentHierarchyDesc*> getAttachmentUiInfo(TendonAttachmentMap& attachmentMap,
                                                                     SpatialTendonVector& spatialTendons);
} // namespace usdparser
} // namespace physx
} // namespace omni
