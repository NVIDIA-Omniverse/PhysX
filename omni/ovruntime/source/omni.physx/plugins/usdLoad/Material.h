// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>


namespace omni
{
namespace physics
{
namespace schema
{
struct MaterialDesc;
}
} // namespace physics

namespace physx
{
namespace usdparser
{

class AttachedStage;

void parseMaterialForPrim(const PXR_NS::UsdPrim& usdPrim, const PXR_NS::SdfPath& materialPath, PhysxMaterialDesc& desc);
void parseDeformableMaterialForPrim(const PXR_NS::UsdPrim& usdPrim,
                                    const PXR_NS::SdfPath& materialPath,
                                    PhysxDeformableMaterialDesc& desc);
void parseSurfaceDeformableMaterialForPrim(const PXR_NS::UsdPrim& usdPrim,
                                           const PXR_NS::SdfPath& materialPath,
                                           PhysxSurfaceDeformableMaterialDesc& desc);
void parsePBDMaterialForPrim(const PXR_NS::UsdPrim& usdPrim, const PXR_NS::SdfPath& materialPath, PBDMaterialDesc& desc);
PhysxMaterialDesc* parseMaterialDesc(const PXR_NS::UsdStageWeakPtr stage, const omni::physics::schema::MaterialDesc& desc);
PhysxDeformableMaterialDesc* parseDeformableMaterialDesc(const PXR_NS::UsdStageWeakPtr stage,
                                                         const omni::physics::schema::DeformableMaterialDesc& desc);
ObjectId getMaterial(AttachedStage& attachedStage, const PXR_NS::SdfPath& path);
ObjectId getMaterial(AttachedStage& attachedStage,
                     const PXR_NS::SdfPath& path,
                     const omni::physx::usdparser::ObjectCategory objCategory);
void setToDefault(PhysxMaterialDesc& desc);
void setToDefault(PXR_NS::UsdStageWeakPtr stage, PhysxDeformableMaterialDesc& desc);
void setToDefault(PXR_NS::UsdStageWeakPtr stage, PhysxSurfaceDeformableMaterialDesc& desc);
void setToDefault(PBDMaterialDesc& desc);

} // namespace usdparser
} // namespace physx
} // namespace omni
