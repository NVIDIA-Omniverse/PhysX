// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
namespace parse
{
struct ObjectKey;
struct SourceUnits;
}
} // namespace physics

namespace physx
{
namespace usdparser
{

class AttachedStage;

void parseMaterialForPrim(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& materialKey, PhysxMaterialDesc& desc);
void parseMaterialForPrim(AttachedStage& attachedStage,
                          const omni::physics::parse::ObjectKey& materialKey,
                          PhysxMaterialDesc& desc);
void parseDeformableMaterialForPrim(PXR_NS::UsdStageWeakPtr stage,
                                    const PXR_NS::SdfPath& materialKey,
                                    PhysxDeformableMaterialDesc& desc);
void parseDeformableMaterialForPrim(AttachedStage& attachedStage,
                                    const omni::physics::parse::ObjectKey& materialKey,
                                    PhysxDeformableMaterialDesc& desc);
void parseSurfaceDeformableMaterialForPrim(PXR_NS::UsdStageWeakPtr stage,
                                           const PXR_NS::SdfPath& materialKey,
                                           PhysxSurfaceDeformableMaterialDesc& desc);
void parseSurfaceDeformableMaterialForPrim(AttachedStage& attachedStage,
                                           const omni::physics::parse::ObjectKey& materialKey,
                                           PhysxSurfaceDeformableMaterialDesc& desc);
void parsePBDMaterialForPrim(PXR_NS::UsdStageWeakPtr stage, const PXR_NS::SdfPath& materialKey, PBDMaterialDesc& desc);
void parsePBDMaterialForPrim(AttachedStage& attachedStage,
                             const omni::physics::parse::ObjectKey& materialKey,
                             PBDMaterialDesc& desc);
ObjectId getMaterial(AttachedStage& attachedStage, const PXR_NS::SdfPath& path);
ObjectId getMaterial(AttachedStage& attachedStage,
                     const PXR_NS::SdfPath& path,
                     const omni::physx::usdparser::ObjectCategory objCategory);
void setToDefault(PhysxMaterialDesc& desc);
// Units-based defaults (backend-agnostic — no USD stage).
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxDeformableMaterialDesc& desc);
void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxSurfaceDeformableMaterialDesc& desc);
void setToDefault(PBDMaterialDesc& desc);

} // namespace usdparser
} // namespace physx
} // namespace omni
