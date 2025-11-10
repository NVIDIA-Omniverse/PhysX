// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

void parseMaterialForPrim(const pxr::UsdPrim& usdPrim, const pxr::SdfPath& materialPath, PhysxMaterialDesc& desc);
void parseDeformableMaterialForPrim(const pxr::UsdPrim& usdPrim,
                                    const pxr::SdfPath& materialPath,
                                    PhysxDeformableMaterialDesc& desc);
void parseSurfaceDeformableMaterialForPrim(const pxr::UsdPrim& usdPrim,
                                           const pxr::SdfPath& materialPath,
                                           PhysxSurfaceDeformableMaterialDesc& desc);
void parsePBDMaterialForPrim(const pxr::UsdPrim& usdPrim, const pxr::SdfPath& materialPath, PBDMaterialDesc& desc);
void parseDeformableBodyMaterialForPrimDeprecated(const pxr::UsdPrim& usdPrim,
                                                  const pxr::SdfPath& materialPath,
                                                  FemSoftBodyMaterialDesc& desc);
void parseDeformableSurfaceMaterialForPrimDeprecated(const pxr::UsdPrim& usdPrim,
                                                     const pxr::SdfPath& materialPath,
                                                     FemClothMaterialDesc& desc);
PhysxMaterialDesc* parseMaterialDesc(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::MaterialDesc& desc);
PhysxDeformableMaterialDesc* parseDeformableMaterialDesc(const pxr::UsdStageWeakPtr stage,
                                                         const omni::physics::schema::DeformableMaterialDesc& desc);
ObjectId getMaterial(AttachedStage& attachedStage, const pxr::SdfPath& path);
ObjectId getMaterial(AttachedStage& attachedStage,
                     const pxr::SdfPath& path,
                     const omni::physx::usdparser::ObjectCategory objCategory);
void setToDefault(PhysxMaterialDesc& desc);
void setToDefault(pxr::UsdStageWeakPtr stage, PhysxDeformableMaterialDesc& desc);
void setToDefault(pxr::UsdStageWeakPtr stage, PhysxSurfaceDeformableMaterialDesc& desc);
void setToDefault(PBDMaterialDesc& desc);
void setToDefaultDeprecated(FemSoftBodyMaterialDesc& desc);
void setToDefaultDeprecated(FemClothMaterialDesc& desc);

} // namespace usdparser
} // namespace physx
} // namespace omni
